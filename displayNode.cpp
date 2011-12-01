static WbDeviceTag display;

//wb_camera_get_image(camera);
uint const displaySize = 120;
double const displayScale = displaySize / 6.0;
wb_display_set_color(display, 0x000000);
wb_display_fill_rectangle(display, 0, 0, displaySize, displaySize);
wb_display_set_color(display, 0xff0000);
wb_display_draw_pixel(display, displayScale * m_pose.m_pos.m_x, displayScale * m_pose.m_pos.m_y);
for (std::deque<NavigationState>::const_iterator i = m_targetQueue.begin(); i != m_targetQueue.end(); ++i)
{
NavigationState const& state = *i;
wb_display_set_color(display, 0x00ff00);
wb_display_draw_pixel(display, displayScale * state.m_targetPos.m_x, displayScale * state.m_targetPos.m_y);  
}
